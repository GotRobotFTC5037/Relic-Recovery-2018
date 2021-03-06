package org.corningrobotics.enderbots.endercv;

import android.content.Context;
import android.view.View;

/**
 * Created by guinea on 6/26/17.
 * Provides methods to display and remove a View from the screen.
 */

public interface ViewDisplay {
    void setCurrentView(Context context, View view);
    void removeCurrentView(Context context);
}
