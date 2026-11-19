import 'package:ebics_setup/app_state.dart';
import 'package:ebics_setup/firmware/firmware_repo.dart';
import 'package:ebics_setup/main.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:material_ui/material_ui.dart';
import 'package:shared_preferences/shared_preferences.dart';

Future<AppState> state() async {
  SharedPreferences.setMockInitialValues({});
  return AppState(await SharedPreferences.getInstance(), FirmwareRepo());
}

void main() {
  testWidgets('home, settings and the first wizard step render', (tester) async {
    tester.view.physicalSize = const Size(1080, 2400);
    tester.view.devicePixelRatio = 3.0; // 360 dp wide: small phone
    addTearDown(tester.view.reset);
    await tester.pumpWidget(EbicsApp(state: await state()));
    await tester.pump();
    expect(find.text('Start first setup'), findsOneWidget);

    await tester.tap(find.text('Settings'));
    await tester.pumpAndSettle();
    expect(find.text('Basic'), findsOneWidget);
    expect(find.text('Write to controller'), findsOneWidget);
    await tester.tap(find.text('Advanced'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Expert'));
    await tester.pumpAndSettle();
    expect(find.textContaining('Sensorless: inductance'), findsOneWidget);
    await tester.pageBack();
    await tester.pumpAndSettle();

    await tester.tap(find.text('Start first setup'));
    await tester.pumpAndSettle();
    expect(find.text('Step 1 of 11'), findsOneWidget);
    expect(find.text('Back'), findsOneWidget);
    expect(find.text('Retry'), findsOneWidget);
    expect(find.text('Next'), findsOneWidget);
  });

  testWidgets('holding a setting shows its explanation, letting go hides it', (tester) async {
    tester.view.physicalSize = const Size(1080, 2400);
    tester.view.devicePixelRatio = 3.0;
    addTearDown(tester.view.reset);
    await tester.pumpWidget(EbicsApp(state: await state()));
    await tester.pump();
    await tester.tap(find.text('Settings'));
    await tester.pumpAndSettle();
    const help = 'Assist stops above this speed.';
    expect(find.textContaining(help), findsNothing);
    // a short tap does not show it
    final field = find.widgetWithText(TextField, 'Speed limit');
    await tester.tap(field);
    await tester.pump(const Duration(seconds: 1));
    expect(find.textContaining(help), findsNothing);
    // holding does
    final g = await tester.startGesture(tester.getCenter(field));
    await tester.pump(const Duration(milliseconds: 600));
    expect(find.textContaining(help), findsOneWidget);
    await g.up();
    await tester.pump();
    expect(find.textContaining(help), findsNothing);
    // also on a switch
    await tester.scrollUntilVisible(find.text('Legal mode (EU)'), 200,
        scrollable: find.descendant(of: find.byType(ListView), matching: find.byType(Scrollable)).first);
    final sw = find.widgetWithText(SwitchListTile, 'Legal mode (EU)');
    final g2 = await tester.startGesture(tester.getCenter(sw));
    await tester.pump(const Duration(milliseconds: 600));
    expect(find.textContaining('limits to 6 km/h'), findsOneWidget);
    await g2.cancel();
    await tester.pump();
    expect(find.textContaining('limits to 6 km/h'), findsNothing);
  });

  testWidgets('bike step validates before saving', (tester) async {
    tester.view.physicalSize = const Size(1080, 2400);
    tester.view.devicePixelRatio = 3.0; // 360 dp wide: small phone
    addTearDown(tester.view.reset);
    final st = await state();
    st.settings['battery'] = 0; // custom battery without voltages = problem
    await tester.pumpWidget(EbicsApp(state: st));
    await tester.tap(find.text('Start first setup'));
    await tester.pumpAndSettle();
    // jump to "Your bike" with Skip (connect + unlock need hardware)
    await tester.tap(find.text('Next'));
    await tester.pump();
    await tester.tap(find.text('Skip'));
    await tester.pump();
    await tester.tap(find.text('Skip'));
    await tester.pumpAndSettle();
    expect(find.text('Your bike'), findsOneWidget);
    await tester.tap(find.text('Next'));
    await tester.pumpAndSettle();
    expect(find.text('Please check'), findsOneWidget);
    await tester.tap(find.text('OK'));
    await tester.pumpAndSettle();
    expect(find.text('Your bike'), findsOneWidget);
    // leave the wizard so the connect step's timers are gone
    await tester.pageBack();
    await tester.pumpAndSettle();
  });
}
